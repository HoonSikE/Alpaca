package com.example.taxi.ui.home.user

import android.annotation.SuppressLint
import android.util.Log
import android.view.View
import androidx.core.content.ContextCompat
import androidx.core.os.bundleOf
import androidx.fragment.app.viewModels
import androidx.navigation.fragment.findNavController
import androidx.recyclerview.widget.LinearLayoutManager
import androidx.recyclerview.widget.RecyclerView
import com.bumptech.glide.Glide
import com.example.taxi.R
import com.example.taxi.base.BaseFragment
import com.example.taxi.data.dto.mypage.Favorites
import com.example.taxi.data.dto.provider.ProviderCar
import com.example.taxi.data.dto.user.destination.Destination
import com.example.taxi.data.dto.user.destination.FrequentDestination
import com.example.taxi.databinding.FragmentUserHomeBinding
import com.example.taxi.di.ApplicationClass
import com.example.taxi.ui.home.provider.ProviderViewModel
import com.example.taxi.utils.constant.UiState
import com.example.taxi.utils.constant.hide
import com.example.taxi.utils.constant.show
import com.example.taxi.utils.view.toast
import dagger.hilt.android.AndroidEntryPoint
import okhttp3.internal.notify

@AndroidEntryPoint
class UserHomeFragment: BaseFragment<FragmentUserHomeBinding>(R.layout.fragment_user_home) {
    private lateinit var destinationListAdapter: DestinationListAdapter
    private lateinit var favoritesAdapter: FavoritesAdapter
    private var favorites : MutableList<Favorites> = mutableListOf()
    private val userHomeViewModel : UserHomeViewModel by viewModels()
    private val providerViewModel : ProviderViewModel by viewModels()
    private lateinit var destination: Destination
    private var addressFavorites = ""

    private val favoritesDeleteClickListener: (View, String) -> Unit = { _, address ->
        addressFavorites = address
        showFavoritesDialog(address)
    }

    private val destinationOnClickListener: (View, String, String, String, String) -> Unit = { _, place, address, x, y ->
        destination = Destination(address,x,place,y)
        findNavController().navigate(R.id.action_userHomeFragment_to_destinationSettingFragment, bundleOf("Destination" to destination))
    }

    private val favoritesOnClickListener: (View, String, String, String, String) -> Unit = { _, place, address, x, y ->
        destination = Destination(address,x,place,y)
        findNavController().navigate(R.id.action_userHomeFragment_to_destinationSettingFragment, bundleOf("Destination" to destination))
    }

    override fun init() {
        initData()
        initAdapter()
        observerData()
        setOnClickListeners()
    }

    private fun initData() {
        val profileImage = ApplicationClass.prefs?.profileImage
        if (profileImage != "")
            Glide.with(this).load(profileImage).into(binding.imageUserHomeProfile)

        binding.textUserHomeName.text = ApplicationClass.prefs.name + "님, 안녕하세요!"
        binding.textUserHomeCount.text = ApplicationClass.prefs.useCount.toString()

        val useCount = ApplicationClass.prefs.useCount
        if (useCount != null) {
            getGrade(useCount)
        }
        if(ApplicationClass.prefs.isEachProvider == true){
            providerViewModel.getProvider()
        }
    }


    private fun initAdapter() {
        userHomeViewModel.getDestinations()
        destinationListAdapter = DestinationListAdapter().apply {
            onItemClickListener = destinationOnClickListener
        }
        binding.recyclerviewUserHomeDestinationList.layoutManager = LinearLayoutManager(requireContext())
        binding.recyclerviewUserHomeDestinationList.adapter = destinationListAdapter
        userHomeViewModel.getFavorites()
        favoritesAdapter = FavoritesAdapter().apply {
            onItemClickListener = favoritesDeleteClickListener
            onFavoritesClickListener = favoritesOnClickListener
        }
        binding.recyclerviewUserHomeFavorites.apply {
            adapter = favoritesAdapter
            layoutManager = LinearLayoutManager(requireContext(), RecyclerView.VERTICAL, false)
        }
    }

    private fun observerData() {
        providerViewModel.provider.observe(viewLifecycleOwner){ state ->
            when (state) {
                is UiState.Loading -> {
                    binding.progressBar.show()
                }
                is UiState.Failure -> {
                    binding.progressBar.hide()
                    state.error?.let {
                        toast(it)
                        Log.d("UiState.Failure", it)
                    }
                }
                is UiState.Success -> {
                    binding.progressBar.hide()
                    val provider = state.data
                    ApplicationClass.prefs.carImage = provider.car!!.carImage
                    ApplicationClass.prefs.carName = provider.car!!.carName
                    ApplicationClass.prefs.carNumber = provider.car!!.carNumber
                }
            }
        }
        userHomeViewModel.destinations.observe(viewLifecycleOwner) { state ->
            when (state) {
                is UiState.Loading -> {
//                    binding.progressBar.show()
                    binding.textUserHomeNoContentDestination.show()
                    binding.textUserHomeFailedDestination.hide()
                }
                is UiState.Failure -> {
//                    binding.progressBar.hide()
                    state.error?.let {
                        toast(it)
                        Log.d("UiState.Failure", it)
                    }
                    binding.recyclerviewUserHomeDestinationList.setBackgroundResource(R.drawable.layout_recycler_no_item)
                    binding.textUserHomeNoContentDestination.hide()
                    binding.textUserHomeFailedDestination.show()
                }
                is UiState.Success -> {
//                    binding.progressBar.hide()
                    Glide.with(requireContext())
                        .load(ApplicationClass.prefs.profileImage)
                        .into(binding.imageUserHomeProfile)
                    binding.textUserHomeName.text = ApplicationClass.prefs.name + "님, 안녕하세요"
                    binding.textUserHomeCount.text = ApplicationClass.prefs.useCount.toString() + "회"
                    val list : MutableList<FrequentDestination> = state.data.toMutableList()
                    destinationListAdapter.updateList(list)
                    binding.recyclerviewUserHomeDestinationList.setBackgroundResource(R.drawable.layout_recycler)
                    binding.textUserHomeNoContentDestination.hide()
                    binding.textUserHomeFailedDestination.hide()
                }
            }
        }
        userHomeViewModel.favorites.observe(viewLifecycleOwner) { state ->
            when (state) {
                is UiState.Loading -> {
//                    binding.progressBar.show()
                    binding.textUserHomeNoContentFavorites.show()
                    binding.textUserHomeFailedFavorites.hide()
                }
                is UiState.Failure -> {
//                    binding.progressBar.hide()
                    state.error?.let {
                        toast(it)
                        Log.d("UiState.Failure", it)
                    }
                    binding.recyclerviewUserHomeFavorites.setBackgroundResource(R.drawable.layout_recycler_no_item)
                    binding.textUserHomeNoContentFavorites.hide()
                    binding.textUserHomeFailedFavorites.show()
                }
                is UiState.Success -> {
//                    binding.progressBar.hide()
                    favorites = state.data.toMutableList()
                    favoritesAdapter.updateList(favorites)
                    binding.recyclerviewUserHomeFavorites.setBackgroundResource(R.drawable.layout_recycler)
                    binding.textUserHomeNoContentFavorites.hide()
                    binding.textUserHomeFailedFavorites.hide()
                }
            }
        }
    }

    private fun setOnClickListeners(){
        binding.imageMoveMyPage.setOnClickListener{
            findNavController().navigate(R.id.action_userHomeFragment_to_myPageFragment)
        }
        binding.imageMoveProvider.setOnClickListener {
            if(ApplicationClass.prefs.isEachProvider == true){
                findNavController().navigate(R.id.action_userHomeFragment_to_providerHomeFragment)
            }else{
                findNavController().navigate(R.id.action_userHomeFragment_to_joinProviderFragment)
            }
        }
        binding.buttonUserHomeCallTaxi.setOnClickListener {
            findNavController().navigate(R.id.action_userHomeFragment_to_startPointSettingFragment)
        }
    }

    private fun showFavoritesDialog(address: String) {
        FavoritesDialogFragment(address){userHomeViewModel.getFavorites()}.show(childFragmentManager, "FAVORITES_DIALOG")
    }

    private fun getGrade(useCount : Int){
        if(0 < useCount && useCount < 5) {
            binding.textUserHomeClass.setText("Bronze")
            binding.textUserHomeClass.setTextColor(ContextCompat.getColor(requireContext(),R.color.bronze))
        }else if(5 <= useCount && useCount < 10){
            binding.textUserHomeClass.setText("Siver")
            binding.textUserHomeClass.setTextColor(ContextCompat.getColor(requireContext(), R.color.silver))
        }else if(10 <= useCount && useCount < 20){
            binding.textUserHomeClass.setText("Gold")
            binding.textUserHomeClass.setTextColor(ContextCompat.getColor(requireContext(), R.color.gold))
        }else if(20 <= useCount && useCount < 30){
            binding.textUserHomeClass.setText("Platinum")
            binding.textUserHomeClass.setTextColor(ContextCompat.getColor(requireContext(), R.color.platinum))
        }else if(useCount >= 30){
            binding.textUserHomeClass.setText("Diamond")
            binding.textUserHomeClass.setTextColor(ContextCompat.getColor(requireContext(), R.color.diamond))
        }else{
            binding.textUserHomeClass.setText("Unrank")
            binding.textUserHomeClass.setTextColor(ContextCompat.getColor(requireContext(), R.color.black))
        }
    }
}