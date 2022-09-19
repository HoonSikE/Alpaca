package com.example.taxi.ui.home.user

import android.view.LayoutInflater
import android.view.ViewGroup
import androidx.recyclerview.widget.RecyclerView
import com.example.taxi.data.dto.user.destination.Destination
import com.example.taxi.databinding.ItemDestinationListBinding


class DestinationListAdapter: RecyclerView.Adapter<DestinationListAdapter.DestinationListViewHolder>() {
    private var destinationList = mutableListOf<Destination>()

    fun setListData(data: MutableList<Destination>){
        destinationList = data
    }

    fun updateList(list: MutableList<Destination>){
        this.destinationList = list
        notifyDataSetChanged()
    }

    override fun onCreateViewHolder(parent: ViewGroup, viewType: Int): DestinationListViewHolder {
        return DestinationListViewHolder(
            ItemDestinationListBinding.inflate(
                LayoutInflater.from(parent.context),
                parent,
                false
            )
        )
    }

    override fun onBindViewHolder(holder: DestinationListViewHolder, position: Int) {
        holder.bind(destinationList[position])
    }

    override fun getItemCount(): Int {
        return destinationList.size
    }

    class DestinationListViewHolder(private val binding: ItemDestinationListBinding) :
        RecyclerView.ViewHolder(binding.root) {

        lateinit var address : String
        lateinit var latitude : String
        lateinit var longitude : String
        var count : Int = 0

        fun bind(data: Destination) {
            binding.addrName.text = data.addressName
            address = data.address
            latitude = data.latitude
            longitude = data.longitude
            count = data.count
        }

//        fun bindOnItemClickListener(onItemClickListener: (View, Int, Int) -> Unit ) {
//            binding.root.setOnClickListener {
//                onItemClickListener(it, binding.tripFollowSelect!!.trip_place_seq, bindingAdapterPosition)
//            }
//        }
    }

}